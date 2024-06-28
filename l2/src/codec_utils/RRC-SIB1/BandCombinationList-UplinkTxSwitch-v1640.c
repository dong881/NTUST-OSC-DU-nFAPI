/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "BandCombinationList-UplinkTxSwitch-v1640.h"

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_BandCombinationList_UplinkTxSwitch_v1640_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 16, -1,  1,  65536 }	/* (SIZE(1..65536)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
asn_TYPE_member_t asn_MBR_BandCombinationList_UplinkTxSwitch_v1640_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_BandCombination_UplinkTxSwitch_v1640,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_BandCombinationList_UplinkTxSwitch_v1640_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_SET_OF_specifics_t asn_SPC_BandCombinationList_UplinkTxSwitch_v1640_specs_1 = {
	sizeof(struct BandCombinationList_UplinkTxSwitch_v1640),
	offsetof(struct BandCombinationList_UplinkTxSwitch_v1640, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
asn_TYPE_descriptor_t asn_DEF_BandCombinationList_UplinkTxSwitch_v1640 = {
	"BandCombinationList-UplinkTxSwitch-v1640",
	"BandCombinationList-UplinkTxSwitch-v1640",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_BandCombinationList_UplinkTxSwitch_v1640_tags_1,
	sizeof(asn_DEF_BandCombinationList_UplinkTxSwitch_v1640_tags_1)
		/sizeof(asn_DEF_BandCombinationList_UplinkTxSwitch_v1640_tags_1[0]), /* 1 */
	asn_DEF_BandCombinationList_UplinkTxSwitch_v1640_tags_1,	/* Same as above */
	sizeof(asn_DEF_BandCombinationList_UplinkTxSwitch_v1640_tags_1)
		/sizeof(asn_DEF_BandCombinationList_UplinkTxSwitch_v1640_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_BandCombinationList_UplinkTxSwitch_v1640_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_BandCombinationList_UplinkTxSwitch_v1640_1,
	1,	/* Single element */
	&asn_SPC_BandCombinationList_UplinkTxSwitch_v1640_specs_1	/* Additional specs */
};

