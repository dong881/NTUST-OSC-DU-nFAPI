/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "FilterConfigCLI-r16.h"

static int asn_DFL_2_cmp_4(const void *sptr) {
	const FilterCoefficient_t *st = sptr;
	
	if(!st) {
		return -1; /* No value is not a default value */
	}
	
	/* Test default value 4 */
	return (*st != 4);
}
static int asn_DFL_2_set_4(void **sptr) {
	FilterCoefficient_t *st = *sptr;
	
	if(!st) {
		st = (*sptr = CALLOC(1, sizeof(*st)));
		if(!st) return -1;
	}
	
	/* Install default value 4 */
	*st = 4;
	return 0;
}
static int asn_DFL_3_cmp_4(const void *sptr) {
	const FilterCoefficient_t *st = sptr;
	
	if(!st) {
		return -1; /* No value is not a default value */
	}
	
	/* Test default value 4 */
	return (*st != 4);
}
static int asn_DFL_3_set_4(void **sptr) {
	FilterCoefficient_t *st = *sptr;
	
	if(!st) {
		st = (*sptr = CALLOC(1, sizeof(*st)));
		if(!st) return -1;
	}
	
	/* Install default value 4 */
	*st = 4;
	return 0;
}
asn_TYPE_member_t asn_MBR_FilterConfigCLI_r16_1[] = {
	{ ATF_POINTER, 2, offsetof(struct FilterConfigCLI_r16, filterCoefficientSRS_RSRP_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FilterCoefficient,
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
		&asn_DFL_2_cmp_4,	/* Compare DEFAULT 4 */
		&asn_DFL_2_set_4,	/* Set DEFAULT 4 */
		"filterCoefficientSRS-RSRP-r16"
		},
	{ ATF_POINTER, 1, offsetof(struct FilterConfigCLI_r16, filterCoefficientCLI_RSSI_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FilterCoefficient,
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
		&asn_DFL_3_cmp_4,	/* Compare DEFAULT 4 */
		&asn_DFL_3_set_4,	/* Set DEFAULT 4 */
		"filterCoefficientCLI-RSSI-r16"
		},
};
static const int asn_MAP_FilterConfigCLI_r16_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_FilterConfigCLI_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_FilterConfigCLI_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* filterCoefficientSRS-RSRP-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* filterCoefficientCLI-RSSI-r16 */
};
asn_SEQUENCE_specifics_t asn_SPC_FilterConfigCLI_r16_specs_1 = {
	sizeof(struct FilterConfigCLI_r16),
	offsetof(struct FilterConfigCLI_r16, _asn_ctx),
	asn_MAP_FilterConfigCLI_r16_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_FilterConfigCLI_r16_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_FilterConfigCLI_r16 = {
	"FilterConfigCLI-r16",
	"FilterConfigCLI-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_FilterConfigCLI_r16_tags_1,
	sizeof(asn_DEF_FilterConfigCLI_r16_tags_1)
		/sizeof(asn_DEF_FilterConfigCLI_r16_tags_1[0]), /* 1 */
	asn_DEF_FilterConfigCLI_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_FilterConfigCLI_r16_tags_1)
		/sizeof(asn_DEF_FilterConfigCLI_r16_tags_1[0]), /* 1 */
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
	asn_MBR_FilterConfigCLI_r16_1,
	2,	/* Elements count */
	&asn_SPC_FilterConfigCLI_r16_specs_1	/* Additional specs */
};

