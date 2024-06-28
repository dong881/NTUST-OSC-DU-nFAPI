/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "FreqBandInformationEUTRA.h"

asn_TYPE_member_t asn_MBR_FreqBandInformationEUTRA_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct FreqBandInformationEUTRA, bandEUTRA),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FreqBandIndicatorEUTRA,
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
		"bandEUTRA"
		},
	{ ATF_POINTER, 2, offsetof(struct FreqBandInformationEUTRA, ca_BandwidthClassDL_EUTRA),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CA_BandwidthClassEUTRA,
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
		"ca-BandwidthClassDL-EUTRA"
		},
	{ ATF_POINTER, 1, offsetof(struct FreqBandInformationEUTRA, ca_BandwidthClassUL_EUTRA),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CA_BandwidthClassEUTRA,
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
		"ca-BandwidthClassUL-EUTRA"
		},
};
static const int asn_MAP_FreqBandInformationEUTRA_oms_1[] = { 1, 2 };
static const ber_tlv_tag_t asn_DEF_FreqBandInformationEUTRA_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_FreqBandInformationEUTRA_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* bandEUTRA */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* ca-BandwidthClassDL-EUTRA */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* ca-BandwidthClassUL-EUTRA */
};
asn_SEQUENCE_specifics_t asn_SPC_FreqBandInformationEUTRA_specs_1 = {
	sizeof(struct FreqBandInformationEUTRA),
	offsetof(struct FreqBandInformationEUTRA, _asn_ctx),
	asn_MAP_FreqBandInformationEUTRA_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_FreqBandInformationEUTRA_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_FreqBandInformationEUTRA = {
	"FreqBandInformationEUTRA",
	"FreqBandInformationEUTRA",
	&asn_OP_SEQUENCE,
	asn_DEF_FreqBandInformationEUTRA_tags_1,
	sizeof(asn_DEF_FreqBandInformationEUTRA_tags_1)
		/sizeof(asn_DEF_FreqBandInformationEUTRA_tags_1[0]), /* 1 */
	asn_DEF_FreqBandInformationEUTRA_tags_1,	/* Same as above */
	sizeof(asn_DEF_FreqBandInformationEUTRA_tags_1)
		/sizeof(asn_DEF_FreqBandInformationEUTRA_tags_1[0]), /* 1 */
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
	asn_MBR_FreqBandInformationEUTRA_1,
	3,	/* Elements count */
	&asn_SPC_FreqBandInformationEUTRA_specs_1	/* Additional specs */
};

