/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "ULTxSwitchingBandPair-v1700.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_uplinkTxSwitchingPeriod2T2T_r17_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_uplinkTxSwitchingPeriod2T2T_r17_value2enum_2[] = {
	{ 0,	5,	"n35us" },
	{ 1,	6,	"n140us" },
	{ 2,	6,	"n210us" }
};
static const unsigned int asn_MAP_uplinkTxSwitchingPeriod2T2T_r17_enum2value_2[] = {
	1,	/* n140us(1) */
	2,	/* n210us(2) */
	0	/* n35us(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_uplinkTxSwitchingPeriod2T2T_r17_specs_2 = {
	asn_MAP_uplinkTxSwitchingPeriod2T2T_r17_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_uplinkTxSwitchingPeriod2T2T_r17_enum2value_2,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_2 = {
	"uplinkTxSwitchingPeriod2T2T-r17",
	"uplinkTxSwitchingPeriod2T2T-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_tags_2,
	sizeof(asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_tags_2)
		/sizeof(asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_tags_2[0]) - 1, /* 1 */
	asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_tags_2,	/* Same as above */
	sizeof(asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_tags_2)
		/sizeof(asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_uplinkTxSwitchingPeriod2T2T_r17_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_uplinkTxSwitchingPeriod2T2T_r17_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_ULTxSwitchingBandPair_v1700_1[] = {
	{ ATF_POINTER, 1, offsetof(struct ULTxSwitchingBandPair_v1700, uplinkTxSwitchingPeriod2T2T_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_uplinkTxSwitchingPeriod2T2T_r17_2,
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
		"uplinkTxSwitchingPeriod2T2T-r17"
		},
};
static const int asn_MAP_ULTxSwitchingBandPair_v1700_oms_1[] = { 0 };
static const ber_tlv_tag_t asn_DEF_ULTxSwitchingBandPair_v1700_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ULTxSwitchingBandPair_v1700_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* uplinkTxSwitchingPeriod2T2T-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_ULTxSwitchingBandPair_v1700_specs_1 = {
	sizeof(struct ULTxSwitchingBandPair_v1700),
	offsetof(struct ULTxSwitchingBandPair_v1700, _asn_ctx),
	asn_MAP_ULTxSwitchingBandPair_v1700_tag2el_1,
	1,	/* Count of tags in the map */
	asn_MAP_ULTxSwitchingBandPair_v1700_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ULTxSwitchingBandPair_v1700 = {
	"ULTxSwitchingBandPair-v1700",
	"ULTxSwitchingBandPair-v1700",
	&asn_OP_SEQUENCE,
	asn_DEF_ULTxSwitchingBandPair_v1700_tags_1,
	sizeof(asn_DEF_ULTxSwitchingBandPair_v1700_tags_1)
		/sizeof(asn_DEF_ULTxSwitchingBandPair_v1700_tags_1[0]), /* 1 */
	asn_DEF_ULTxSwitchingBandPair_v1700_tags_1,	/* Same as above */
	sizeof(asn_DEF_ULTxSwitchingBandPair_v1700_tags_1)
		/sizeof(asn_DEF_ULTxSwitchingBandPair_v1700_tags_1[0]), /* 1 */
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
		SEQUENCE_constraint
	},
	asn_MBR_ULTxSwitchingBandPair_v1700_1,
	1,	/* Elements count */
	&asn_SPC_ULTxSwitchingBandPair_v1700_specs_1	/* Additional specs */
};

